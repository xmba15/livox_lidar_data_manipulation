option(WITH_DEBUG "Enable debug" OFF)
if(WITH_DEBUG)
  target_compile_definitions(${PROJECT_TARGET_LIB_NAME}
    PUBLIC
    WITH_DEBUG
  )
endif(WITH_DEBUG)

option(BUILD_TEST "Enable debug" OFF)
if(BUILD_TEST)
  enable_testing()
  add_subdirectory(tests)
endif(BUILD_TEST)

option(BUILD_DOC "Build documentation" OFF)
if(BUILD_DOC)
  find_package(Doxygen QUIET)
  if(DOXYGEN_FOUND)
    set(DOXYGEN_IN ${PROJECT_SOURCE_DIR}/cmake/${PROJECT_TARGET_LIB_NAME}_Doxyfile.in)
    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
    message(STATUS "Doxygen build started")

    add_custom_target(${PROJECT_TARGET_LIB_NAME}_doc_doxygen ALL
      COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
      COMMENT "Generating API documentation with Doxygen"
      VERBATIM
    )
  else()
    message(STATUS "Doxygen not found")
  endif(DOXYGEN_FOUND)
endif(BUILD_DOC)
