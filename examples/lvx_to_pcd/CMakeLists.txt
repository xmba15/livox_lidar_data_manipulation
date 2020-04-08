cmake_minimum_required(VERSION 3.10)

set(APP_NAME lvx_to_pcd_app)

find_package(PCL REQUIRED)

add_executable(${APP_NAME}
  ${CMAKE_CURRENT_LIST_DIR}/main.cpp
  ${CMAKE_CURRENT_LIST_DIR}/../shared_utilities/Utility.cpp
  ${CMAKE_CURRENT_LIST_DIR}/../shared_utilities/TypesConversion.cpp
)

target_link_libraries(${APP_NAME}
  PRIVATE
    ${PCL_LIBRARIES}
    ${PROJECT_TARGET_LIB_NAME}
)

target_include_directories(${APP_NAME}
  PRIVATE
    ${PCL_INCLUDE_DIRS}
)
