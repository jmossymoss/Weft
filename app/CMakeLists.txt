# Interactive shell. Needs GLFW + OpenGL + Dear ImGui. System packages are
# used when present (Debian/Ubuntu: libglfw3-dev libgl1-mesa-dev
# libimgui-dev); otherwise GLFW and ImGui are fetched and built from source
# (the normal path on Windows). stb_image_write is vendored in third_party/.
# Only OpenGL itself is a hard requirement.
find_package(OpenGL QUIET)
if(NOT OpenGL_FOUND)
  message(STATUS "weft_app skipped (no OpenGL)")
  return()
endif()

option(WEFT_FETCH_APP_DEPS
       "Force-fetch GLFW/ImGui from source even if installed" OFF)

if(NOT WEFT_FETCH_APP_DEPS)
  find_package(glfw3 3.3 QUIET)
endif()

# ImGui is ALWAYS fetched, pinned to the DOCKING build: dockable panels
# need it and no distro ships the docking branch. GLFW still comes from
# the system when present.
#
# Use GitHub release/tag zip archives (URL) rather than GIT_REPOSITORY so
# configure works without git on PATH — the common Windows failure mode
# when only CMake + VS Build Tools are installed.
if(TRUE)
  include(FetchContent)
  if(POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)
  endif()
  message(STATUS "weft_app: fetching ImGui (docking) sources")

  if(NOT glfw3_FOUND)
    set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(glfw
      URL https://github.com/glfw/glfw/archive/refs/tags/3.4.zip
      URL_HASH SHA256=a133ddc3d3c66143eba9035621db8e0bcf34dba1ee9514a9e23e96afd39fd57a
    )
    FetchContent_MakeAvailable(glfw)
  endif()

  FetchContent_Declare(imgui
    URL https://github.com/ocornut/imgui/archive/refs/tags/v1.90.9-docking.zip
    URL_HASH SHA256=1dba00780ccd40d90fb423adb3f193424083b024b6cd347e27ec4af47a41de1a
  )
  FetchContent_MakeAvailable(imgui)

  add_library(weft_imgui STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
  )
  target_include_directories(weft_imgui PUBLIC ${imgui_SOURCE_DIR})

  set(WEFT_IMGUI_LIB weft_imgui)
  set(WEFT_IMGUI_INC ${imgui_SOURCE_DIR})
  # Use the backends that ship with the fetched ImGui, so versions match.
  set(WEFT_BACKEND_SRC
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
  )
endif()

add_executable(weft_app main.cpp ${WEFT_BACKEND_SRC})

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
