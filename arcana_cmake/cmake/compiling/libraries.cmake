# Copyright (C) 2024-2025 by Meltwin
# Authors:
#   Geoffrey Côte: github@meltwin.fr

# The MIT License (MIT)  https://mit-license.org/

# Permission is hereby granted, free of charge, to any person obtaining a copy of this software
# and associated documentation files (the “Software”), to deal in the Software without
# restriction, including without limitation the rights to use, copy, modify, merge, publish,
# distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all copies
# or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
# INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
# PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
# FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
# ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

find_package(ament_cmake_auto)
include("${CMAKE_CURRENT_LIST_DIR}/../utils/simple.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../utils/files.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../utils/display.cmake")

# This function lets user define library through ament while being able to export them
# for other packages usage.
function(arcana_add_library _target)
  cmake_parse_arguments(ARG "STATIC;AUTO" "DIR;NAMESPACE" "FILES;INCLUDE;DEPS" ${ARGN})

  # Whether we generate a static or shared library
  set(link_type "PUBLIC")
  set(ament_link_type "SYSTEM")
  ternary(lib_type ARG_STATIC "STATIC" "SHARED")
  # Define alias if needed
  if (NOT "${ARG_NAMESPACE}" STREQUAL "")
    set(to_export ${ARG_NAMESPACE}::${_target})
  else()
    set(to_export ${_target})
  endif()

  # Get library sources
  # If no source files found, make the library an interface instead
  set(is_interface OFF)
  arcana_fetch_sources(lib_src DIRS ${ARG_DIR} FILES ${ARG_FILES})
  if ("${lib_src}" STREQUAL "")
    message(WARNING "No source file found for ${_target}, switching to INTERFACE instead")
    set(lib_type "INTERFACE")
    set(link_type "INTERFACE")
    set(ament_link_type "INTERFACE")
  endif()

  # Declare library
  dispStep("Declaring ${lib_type} library \"${_target}\"")

  dispLine("Linking libraries / includes")
  if (ARG_AUTO)
    ament_auto_add_library(${_target} ${lib_type} ${lib_src})
    target_link_libraries(${_target} ${ARG_DEPS} ${${PROJECT_NAME}_CUSTOM_MSGS_LIB})
    target_include_directories(${_target} ${link_type} ${ARG_INCLUDE})
  else()
    add_library(${_target} ${lib_type} ${lib_src})
    ament_target_dependencies(${_target} SYSTEM ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})
    target_link_libraries(${_target} ${ARG_DEPS} ${${PROJECT_NAME}_CUSTOM_MSGS_LIB})
    target_include_directories(${_target} ${link_type} ${ARG_INCLUDE})
    list(APPEND ${PROJECT_NAME}_LIBRARIES ${_target})
  endif()

  # Export the library
  if(NOT ${lib_type} STREQUAL "INTERFACE")
    dispLine("Exporting library")
    install(
      TARGETS ${_target} EXPORT ${_target}
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
    )
    ament_export_libraries(${_target})
    ament_export_targets(${_target} HAS_LIBRARY_TARGET NAMESPACE ${ARG_NAMESPACE})
  endif()

  # Add to the package's library register
  set(${PROJECT_NAME}_LIBRARIES ${${PROJECT_NAME}_LIBRARIES} PARENT_SCOPE)
endfunction()

# Wrapper around the arcana_add_library to add all project dependencies
function(arcana_auto_add_library _target)
  cmake_parse_arguments(_FUNC_ARG "AUTO" "" "" ${ARGN})
  arcana_add_library(${_target} AUTO ${_FUNC_ARG_UNPARSED_ARGUMENTS})
endfunction()