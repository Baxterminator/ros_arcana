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

function(arcana_install_include)
  cmake_parse_arguments(_FUNC_ARG "NO_PROJECT_NAME" "" "TOP_DIRS;DIRS" ${ARGN})

  # Macro to register the found include directories
  dispSection("Exporting includes directory" PARENT)
  set(includes_dir "")
  macro(register_include directory)
    if (_FUNC_ARG_NO_PROJECT_NAME)
      set(include_name ${directory})
    else()
      set(include_name "${directory}/${PROJECT_NAME}")
    endif()
    dispStep("Registering include directory ${include_name}")
    list(APPEND includes_dir ${include_name})
  endmacro()

  # Fetch all directories in top_dirs
  foreach (top_dir ${_FUNC_ARG_TOP_DIRS})
    file(GLOB inner_dirs LIST_DIRECTORIES true "${top_dir}/*")
    foreach(dir ${inner_dirs})
      register_include(${dir})
    endforeach()
  endforeach()

  # Fetch all regular include directories
  foreach(dir ${_FUNC_ARG_DIRS})
    register_include(${dir})
  endforeach()
  
  # Fetch all manual include directories
  install(
    DIRECTORY ${includes_dir}
    DESTINATION include/${PROJECT_NAME}
)
endfunction()