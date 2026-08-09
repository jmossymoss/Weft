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

find_package(OpenCASCADE REQUIRED)

# build.bat deploys the TK*.dlls next to the exes, but the TK dlls in turn
# load the third-party runtimes OCCT was built against (tbb12, jemalloc,
# FreeImage, openvr_api, freetype, ffmpeg...) and those usually live
# OUTSIDE the OCCT install dir. Their true locations are recorded in
# OCCT's link interface as absolute .lib paths, so resolve each to the
# .dll beside it (or in a sibling bin/) and deploy those too.
if(WIN32)
  set(WEFT_OCCT_3P_DLLS "")
  set(_weft_seen "")
  set(_weft_stack ${OpenCASCADE_LIBRARIES})
  while(_weft_stack)
    list(POP_FRONT _weft_stack _t)
    if(NOT TARGET ${_t})
      continue()
    endif()
    if(_t IN_LIST _weft_seen)
      continue()
    endif()
    list(APPEND _weft_seen ${_t})
    get_target_property(_deps ${_t} INTERFACE_LINK_LIBRARIES)
    if(NOT _deps)
      continue()
    endif()
    foreach(_d IN LISTS _deps)
      if(TARGET ${_d})
        list(APPEND _weft_stack ${_d})
        continue()
      endif()
      # Entries may be wrapped in generator expressions; fish the path out.
      string(REGEX MATCH "[A-Za-z]:[^<>;$]*\\.lib" _lib "${_d}")
      if(NOT _lib)
        continue()
      endif()
      if(NOT EXISTS "${_lib}")
        # OCCT was built against this lib but it is gone from that path
        # (relocated install). Say so: its dll must be found by hand.
        list(APPEND _weft_3p_missing "${_lib}")
        continue()
      endif()
      get_filename_component(_dir "${_lib}" DIRECTORY)
      get_filename_component(_name "${_lib}" NAME_WE)
      # Versioned dll names (zlib1.dll, avcodec-57.dll) need the glob.
      file(GLOB _dll "${_dir}/${_name}*.dll" "${_dir}/../bin/${_name}*.dll")
      list(APPEND WEFT_OCCT_3P_DLLS ${_dll})
    endforeach()
  endwhile()
  list(REMOVE_DUPLICATES WEFT_OCCT_3P_DLLS)
  # Relocated OCCT Windows zips often break the absolute .lib paths in
  # the link interface. Fall back to scanning OCCT_DIR / 3rdparty siblings
  # for the known runtime DLL names (release + debug variants).
  if(NOT WEFT_OCCT_3P_DLLS)
    set(_weft_tp_roots "")
    if(DEFINED OpenCASCADE_INSTALL_PREFIX)
      list(APPEND _weft_tp_roots "${OpenCASCADE_INSTALL_PREFIX}")
      list(APPEND _weft_tp_roots "${OpenCASCADE_INSTALL_PREFIX}/..")
    endif()
    if(DEFINED OpenCASCADE_DIR)
      get_filename_component(_weft_occt_cfg_dir "${OpenCASCADE_DIR}" DIRECTORY)
      list(APPEND _weft_tp_roots "${_weft_occt_cfg_dir}")
      list(APPEND _weft_tp_roots "${_weft_occt_cfg_dir}/..")
      list(APPEND _weft_tp_roots "${_weft_occt_cfg_dir}/../..")
    endif()
    if(DEFINED OCCT_SEARCH_PATH)
      list(APPEND _weft_tp_roots "${OCCT_SEARCH_PATH}")
      list(APPEND _weft_tp_roots "${OCCT_SEARCH_PATH}/..")
    endif()
    list(REMOVE_DUPLICATES _weft_tp_roots)
    set(_weft_tp_names
      tbb12 tbb12_debug tbbmalloc tbbmalloc_debug
      jemalloc jemalloc_debug
      FreeImage FreeImage_debug
      openvr_api openvr_api_debug
      freetype freetype_debug
      zlib1 zlibd1)
    foreach(_root IN LISTS _weft_tp_roots)
      if(NOT EXISTS "${_root}")
        continue()
      endif()
      foreach(_name IN LISTS _weft_tp_names)
        file(GLOB_RECURSE _hit "${_root}/${_name}.dll")
        list(APPEND WEFT_OCCT_3P_DLLS ${_hit})
      endforeach()
    endforeach()
    list(REMOVE_DUPLICATES WEFT_OCCT_3P_DLLS)
  endif()
  if(WEFT_OCCT_3P_DLLS)
    message(STATUS "OCCT third-party runtimes to deploy: ${WEFT_OCCT_3P_DLLS}")
  else()
    message(STATUS "OCCT third-party runtimes: none resolved from the link "
                   "interface (OCCT built without them, or relocated)")
  endif()
  if(_weft_3p_missing)
    list(REMOVE_DUPLICATES _weft_3p_missing)
    message(WARNING "OCCT's link interface records third-party libs at "
                    "paths that no longer exist -- their DLLs must be "
                    "located manually: ${_weft_3p_missing}")
  endif()
endif()

# Copy the third-party DLLs next to a just-built executable. Prefer the
# vendored tree in third_party/occt-win-runtime (checked into git); fall
# back to whatever CMake resolved from OCCT's link interface.
set(WEFT_VENDORED_OCCT_RUNTIME_DIR
    "${CMAKE_SOURCE_DIR}/third_party/occt-win-runtime")
if(WIN32 AND EXISTS "${WEFT_VENDORED_OCCT_RUNTIME_DIR}")
  file(GLOB WEFT_VENDORED_OCCT_RUNTIME_DLLS
       "${WEFT_VENDORED_OCCT_RUNTIME_DIR}/*.dll")
  if(WEFT_VENDORED_OCCT_RUNTIME_DLLS)
    message(STATUS "OCCT third-party runtimes (vendored): "
                   "${WEFT_VENDORED_OCCT_RUNTIME_DIR}")
    set(WEFT_OCCT_3P_DLLS "${WEFT_VENDORED_OCCT_RUNTIME_DLLS}")
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
