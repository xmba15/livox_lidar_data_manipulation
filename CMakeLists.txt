cmake_minimum_required(VERSION 3.8)

project(liblvx_to_pcd)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

set(PROJECT_TARGET_LIB_NAME "lvx_to_pcd")

find_library(LIVOX_SDK_LIBRARY liblivox_sdk_static.a /usr/local/lib)
if((NOT LIVOX_SDK_LIBRARY) OR (NOT EXISTS ${LIVOX_SDK_LIBRARY}))
  include(livox_sdk)
else()
  message("find livox sdk library success")
endif()

include(FindPkgConfig)
set(ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH}:/usr/local/apr/lib/pkgconfig")
pkg_check_modules(LIBAPR REQUIRED apr-1)

find_package(Threads REQUIRED)
add_subdirectory(src)
include(lvx_to_pcd_build_options)
