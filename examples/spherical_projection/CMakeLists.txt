cmake_minimum_required(VERSION 3.10)

set(APP_NAME spherical_projection)

find_package(PCL REQUIRED)
find_package(OpenCV QUIET)

if (NOT ${OpenCV_FOUND})
  message(STATUS "Failed to find OpenCV; stop building\n")
endif()

add_executable(${APP_NAME}
  ${CMAKE_CURRENT_LIST_DIR}/main.cpp
  ${CMAKE_CURRENT_LIST_DIR}/../shared_utilities/Utility.cpp
  ${CMAKE_CURRENT_LIST_DIR}/../shared_utilities/TypesConversion.cpp
)

target_link_libraries(${APP_NAME}
  PUBLIC
    ${PROJECT_TARGET_LIB_NAME}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

target_include_directories(${APP_NAME}
  SYSTEM PUBLIC
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)
