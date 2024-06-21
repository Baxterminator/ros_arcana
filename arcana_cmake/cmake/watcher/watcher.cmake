# Copyright (C) 2024-2024 by Meltwin
# Authors:
#   Geoffrey Côte: geoffrey.cote@centraliens-nantes.org

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

function(setup_project_watcher)
  cmake_parse_arguments(ARG "DISPLAY" "_CUSTOM_ROOT" "FOLDERS" ${ARGN})

  # Process folders arguments
  if (ARG_FOLDERS)
    set(_watcher_folders "-fwl;${ARG_FOLDERS}")
  endif()

  # Whether we want the watcher to log
  if (ARG_DISPLAY)
    list(APPEND _flags "-l")
  endif()

  # Setup a custom root folder for finding the watcher executable
  if (ARG__CUSTOM_ROOT)
    set(_watcher_root "${ARG__CUSTOM_ROOT}")
  else()
    set(_watcher_root "${arcana_cmake_DIR}")
  endif()

  add_custom_target("${PROJECT_NAME}_arcana_watcher" ALL) 
  add_custom_command(
      TARGET "${PROJECT_NAME}_arcana_watcher" PRE_BUILD
      COMMAND python3 "${_watcher_root}/watcher/project_watcher.py" ${CMAKE_CURRENT_LIST_DIR} ${CMAKE_CURRENT_BINARY_DIR} ${_flags} ${_watcher_folders}
  )
endfunction()