cmake_minimum_required(VERSION 3.20)
project(weft VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

# On Windows the executables need the OCCT DLLs next to them; gather all
# targets into one bin/ directory so the build script can deploy the DLLs
# once and everything runs from anywhere (build/bin/Release/*.exe).
if(WIN32)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
endif()

message(STATUS "Looking for OpenCASCADE...")
find_package(OpenCASCADE REQUIRED)
message(STATUS "OpenCASCADE found")

# Third-party OCCT runtimes (tbb12, jemalloc, FreeImage, ...). Prefer the
# vendored tree in git — NEVER recursively walk C:\OpenCASCADE (that
# GLOB_RECURSE was hanging Windows configure for minutes after the
# "Selecting Windows SDK version" line).
set(WEFT_OCCT_3P_DLLS "")
set(WEFT_VENDORED_OCCT_RUNTIME_DIR
    "${CMAKE_SOURCE_DIR}/third_party/occt-win-runtime")
if(WIN32 AND EXISTS "${WEFT_VENDORED_OCCT_RUNTIME_DIR}/tbb12.dll")
  file(GLOB WEFT_VENDORED_OCCT_RUNTIME_DLLS
       "${WEFT_VENDORED_OCCT_RUNTIME_DIR}/*.dll")
  set(WEFT_OCCT_3P_DLLS "${WEFT_VENDORED_OCCT_RUNTIME_DLLS}")
  list(LENGTH WEFT_OCCT_3P_DLLS _weft_n)
  message(STATUS "OCCT third-party runtimes: ${_weft_n} vendored DLL(s) in "
                 "${WEFT_VENDORED_OCCT_RUNTIME_DIR}")
elseif(WIN32)
  message(STATUS "OCCT third-party runtimes: vendored folder missing; "
                 "trying shallow OCCT bin dirs (no recursive scan)")
  # Shallow only: bin/ next to OpenCASCADEConfig.cmake and common 3rdparty
  # layouts. Do not GLOB_RECURSE the whole install tree.
  set(_weft_tp_bins "")
  if(DEFINED OpenCASCADE_DIR)
    get_filename_component(_weft_occt_cfg_dir "${OpenCASCADE_DIR}" DIRECTORY)
    list(APPEND _weft_tp_bins
      "${_weft_occt_cfg_dir}/../win64/vc14/bin"
      "${_weft_occt_cfg_dir}/../bind"
      "${_weft_occt_cfg_dir}/../bin"
      "${_weft_occt_cfg_dir}/../../3rdparty-vc14-64/tbb/bin"
      "${_weft_occt_cfg_dir}/../../3rdparty-vc14-64/freeimage/bin"
      "${_weft_occt_cfg_dir}/../../3rdparty-vc14-64/freetype/bin"
      "${_weft_occt_cfg_dir}/../../3rdparty-vc14-64/openvr/bin/win64"
      "${_weft_occt_cfg_dir}/../../3rdparty-vc14-64/jemalloc-vc14-64/bin")
  endif()
  if(DEFINED OCCT_SEARCH_PATH)
    list(APPEND _weft_tp_bins
      "${OCCT_SEARCH_PATH}/win64/vc14/bin"
      "${OCCT_SEARCH_PATH}/bind"
      "${OCCT_SEARCH_PATH}/bin")
  endif()
  list(REMOVE_DUPLICATES _weft_tp_bins)
  foreach(_bin IN LISTS _weft_tp_bins)
    if(EXISTS "${_bin}")
      file(GLOB _hit "${_bin}/*.dll")
      list(APPEND WEFT_OCCT_3P_DLLS ${_hit})
    endif()
  endforeach()
  list(REMOVE_DUPLICATES WEFT_OCCT_3P_DLLS)
  if(WEFT_OCCT_3P_DLLS)
    list(LENGTH WEFT_OCCT_3P_DLLS _weft_n)
    message(STATUS "OCCT third-party runtimes: ${_weft_n} DLL(s) from shallow bins")
  else()
    message(STATUS "OCCT third-party runtimes: none found — pull "
                   "third_party/occt-win-runtime from git for weft_app.exe")
  endif()
endif()

# Copy the third-party DLLs next to a just-built executable. All exes
# share CMAKE_RUNTIME_OUTPUT_DIRECTORY, so any one build refreshes them.
function(weft_deploy_occt_dlls tgt)
  if(WIN32 AND WEFT_OCCT_3P_DLLS)
    add_custom_command(TARGET ${tgt} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
              ${WEFT_OCCT_3P_DLLS} $<TARGET_FILE_DIR:${tgt}>
      VERBATIM)
  endif()
endfunction()

enable_testing()

add_subdirectory(core)
add_subdirectory(cli)
add_subdirectory(app)
add_subdirectory(tests)
