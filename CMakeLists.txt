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

enable_testing()

add_subdirectory(core)
add_subdirectory(cli)
add_subdirectory(app)
add_subdirectory(tests)
