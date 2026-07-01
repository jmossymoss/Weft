cmake_minimum_required(VERSION 3.20)
project(weft VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(OpenCASCADE REQUIRED)

enable_testing()

add_subdirectory(core)
add_subdirectory(cli)
add_subdirectory(tests)
