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
  find_library(IMGUI_LIB imgui)
  find_path(IMGUI_INCLUDE imgui.h PATH_SUFFIXES imgui)
  # Debian's ImGui is built against the system stb (unbundled), so the
  # stb library must be linked alongside it for the stbtt_* font symbols.
  find_library(STB_LIB stb)
endif()

if(glfw3_FOUND AND IMGUI_LIB AND IMGUI_INCLUDE AND STB_LIB)
  set(WEFT_IMGUI_LIB ${IMGUI_LIB} ${STB_LIB})
  set(WEFT_IMGUI_INC ${IMGUI_INCLUDE})
  # Local copies of the ImGui backends, matching the system ImGui version.
  set(WEFT_BACKEND_SRC
    imgui_backends/imgui_impl_glfw.cpp
    imgui_backends/imgui_impl_opengl3.cpp
  )
else()
  include(FetchContent)
  message(STATUS "weft_app: fetching GLFW/ImGui sources (system packages not found)")

  if(NOT glfw3_FOUND)
    set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(glfw
      URL https://github.com/glfw/glfw/archive/refs/tags/3.4.tar.gz
      DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    FetchContent_MakeAvailable(glfw)
  endif()

  FetchContent_Declare(imgui
    URL https://github.com/ocornut/imgui/archive/refs/tags/v1.90.9.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
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
