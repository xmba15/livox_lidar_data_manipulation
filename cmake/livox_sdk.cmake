message("Coudn't find livox sdk library!")
message("git clone Livox-SDK from github temporarily, only for ROS distro jenkins build!")

# clone livox sdk source code from github
execute_process(COMMAND rm -rf ${PROJECT_SOURCE_DIR}/Livox-SDK OUTPUT_VARIABLE cmd_res)
message("Try to pull the livox sdk source code from github")
FOREACH(res ${cmd_res})
  MESSAGE(${res})
ENDFOREACH()

execute_process(COMMAND git clone https://github.com/Livox-SDK/Livox-SDK.git ${PROJECT_SOURCE_DIR}/Livox-SDK OUTPUT_VARIABLE cmd_res)
FOREACH(res ${cmd_res})
  MESSAGE(${res})
ENDFOREACH()

execute_process(COMMAND cmake .. WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/Livox-SDK/build OUTPUT_VARIABLE cmd_res)
FOREACH(res ${cmd_res})
  MESSAGE(${res})
ENDFOREACH()

execute_process(COMMAND make WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/Livox-SDK/build OUTPUT_VARIABLE cmd_res)
FOREACH(res ${cmd_res})
  MESSAGE(${res})
ENDFOREACH()

include_directories(
  ./${PROJECT_SOURCE_DIR}/Livox-SDK/sdk_core/include
)

link_directories(
  ./${PROJECT_SOURCE_DIR}/Livox-SDK/build/sdk_core
)
