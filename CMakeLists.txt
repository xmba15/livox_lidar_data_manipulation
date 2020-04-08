cmake_minimum_required(VERSION 3.10)

project(liblvx_to_pcd)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

set(PROJECT_TARGET_LIB_NAME "lvx_to_pcd")
set(CMAKE_CXX_STANDARD 17)

find_library(LIVOX_SDK_LIBRARY liblivox_sdk_static.a /usr/local/lib)
if((NOT LIVOX_SDK_LIBRARY) OR (NOT EXISTS ${LIVOX_SDK_LIBRARY}))
  include(livox_sdk)
  link_directories(${LIVOX_SDK_LIBRARY})
else()
  message("find livox sdk library success")
endif()

find_library(APR_LIBRARY libapr-1.a /usr/local/apr/lib)

find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
include(Findrapidxml)

if(NOT RAPIDXML_FOUND)
  message(STATUS "rapidxml not found\n")
  return()
endif()

add_subdirectory(src)
include(lvx_to_pcd_build_options)

option(BUILD_EXAMPLES "Option whether to build examples" OFF)
if(BUILD_EXAMPLES)
  message(STATUS "Also build examples")
  add_subdirectory(examples/lvx_to_pcd)
  add_subdirectory(examples/visualize_lvx)
endif(BUILD_EXAMPLES)
