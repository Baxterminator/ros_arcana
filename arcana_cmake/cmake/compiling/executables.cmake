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
include("${CMAKE_CURRENT_LIST_DIR}/../utils/files.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../utils/display.cmake")

function(arcana_add_executable _target)
  cmake_parse_arguments(ARG "AUTO" "DIR" "FILES;INCLUDE;DEPS;AMENT_DEPS" ${ARGN})

  # Get executable sources
  # If no source files found, make the library an interface instead
  arcana_fetch_sources(exe_src DIRS ${ARG_DIR} FILES ${ARG_FILES})
  if ("${exe_src}" STREQUAL "")
    message(WARNING "No source file found for ${_target}")
    return()
  endif()

  dispStep("Declaring executable \"${_target}\"")
  if (ARG_AUTO)
    ament_auto_add_executable(${_target} ${exe_src})
    dispLine("Linking libraries / includes")
    target_link_libraries(${_target} ${ARG_DEPS} ${${PROJECT_NAME}_CUSTOM_MSGS_LIB})
    target_include_directories(${_target} PUBLIC ${ARG_INCLUDE})
  else()
    add_executable(${_target}  ${exe_src})
    dispLine("Linking libraries / includes")
    ament_target_dependencies(${_target} SYSTEM ${ARG_AMENT_DEPS})
    target_link_libraries(${_target} ${ARG_DEPS} ${${PROJECT_NAME}_CUSTOM_MSGS_LIB})
    target_include_directories(${_target} PUBLIC ${ARG_INCLUDE})
    list(APPEND ${PROJECT_NAME}_EXECUTABLES ${_target})
  endif()

  # Add to the package's executable register
  set(${PROJECT_NAME}_EXECUTABLES ${${PROJECT_NAME}_EXECUTABLES} PARENT_SCOPE)
endfunction()

function(arcana_install_programs)
  dispSection("Exporting scripts" PARENT)
  cmake_parse_arguments(_FUNC_ARG "" "" "" ${ARGN})

  # Fetch scripts
  arcana_fetch_files(programs_files ${_FUNC_ARG_UNPARSED_ARGUMENTS} NO_EXT)
  foreach(script ${programs_files})
    dispStep("Exporting script \"${script}\"")
    install(
      PROGRAMS ${script} 
      DESTINATION lib/${PROJECT_NAME}
    )
  endforeach()
endfunction()