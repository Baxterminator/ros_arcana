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

function(arcana_autogen_rosd)
  cmake_parse_arguments(_FUNC_ARG "" "LANG;BUILD_FILE" "" ${ARGN})
  dispStep("Creating autogen target ${PROJECT_NAME}_rosd_${_FUNC_ARG_LANG}")
  add_custom_target(
    ${PROJECT_NAME}_rosd_${_FUNC_ARG_LANG}
    ALL
    BYPRODUCTS ${_FUNC_ARG_BUILD_FILE}
    COMMAND python3 "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/mk_rosd.py" 
        ${_FUNC_ARG_LANG}  
        "${_FUNC_ARG_BUILD_FILE}" 
        -m "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/rosdistros.yaml"
        -l "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/licence.txt"
  )
endfunction()

function(arcana_autogen_rosd_cmake)
  arcana_autogen_rosd(
    LANG cmake
    BUILD_FILE ${CMAKE_CURRENT_BINARY_DIR}/arcana_autogen/rosd_autogen.cmake
  )
  register_cmake(FILES "${CMAKE_CURRENT_BINARY_DIR}/arcana_autogen/rosd_autogen.cmake" NO_CHECK)
endfunction()

function(arcana_autogen_rosd_cpp)
  arcana_autogen_rosd(
    LANG cpp
    BUILD_FILE ${CMAKE_CURRENT_BINARY_DIR}/arcana_autogen/rosdistro.hpp
  )
  install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/arcana_autogen/rosdistro.hpp
    DESTINATION include/${PROJECT_NAME}
  )
endfunction()

function(arcana_autogen_rosd_py)
  cmake_parse_arguments(_FUNC_ARG "" "DEST" "" ${ARGN})
  arcana_autogen_rosd(
    LANG py
    BUILD_FILE ${PROJECT_SOURCE_DIR}/${_FUNC_ARG_DEST}
  )
endfunction()