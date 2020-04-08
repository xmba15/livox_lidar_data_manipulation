include(FindPackageHandleStandardArgs)
unset(RAPIDXML_FOUND)

find_path(rapidxml_INCLUDE_DIR
  NAMES
    rapidxml.hpp
  HINTS
    /usr/include/rapidxml
    /usr/local/include/rapidxml
)

find_package_handle_standard_args(rapidxml
  DEFAULT_MSG
    rapidxml_INCLUDE_DIR
)

if(RAPIDXML_FOUND)
  set(rapidxml_INCLUDE_DIRS
    ${rapidxml_INCLUDE_DIR}
  )
endif()

if(RAPIDXML_FOUND AND NOT TARGET rapidxml::rapidxml)
    add_library(rapidxml::rapidxml INTERFACE IMPORTED)
    set_target_properties(rapidxml::rapidxml PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${rapidxml_INCLUDE_DIRS}"
    )
endif()

mark_as_advanced(rapidxml_INCLUDE_DIR)
