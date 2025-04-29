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

macro(foreach_file_exist file)
  if (NOT EXISTS ${file})
    message(WARNING "File ${file} does not exist!")
    continue()
  endif()
endmacro()

# Fetch the files with the given extensions from both directories and given 
# files and gather them in one variable.
function(arcana_fetch_files out)
  cmake_parse_arguments(_FUNC_ARG "NO_EXT" "" "DIRS;FILES;EXTS" ${ARGN})
  set(src_files "")

  # Create macro to generate file globing
  macro(reset_filter)
    set(glob_filter "")
  endmacro()
  macro(create_filter d)
    reset_filter()
    if (_FUNC_ARG_NO_EXT)
      set(glob_filter "${d}/**")
    else()
      foreach(e ${_FUNC_ARG_EXTS})
        list(APPEND glob_filter "${d}/**${e}")
      endforeach()
    endif()
  endmacro()
  
  # Fetch from directories
  foreach(dir ${_FUNC_ARG_DIRS})
    create_filter(${dir})
    file(GLOB_RECURSE dir_sources LIST_DIRECTORIES false RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ${glob_filter})
    list(APPEND src_files ${dir_sources})
  endforeach()

  # Add files
  list(APPEND src_files ${_FUNC_ARG_FILES})

  # Return to parent
  set(${out} ${src_files} PARENT_SCOPE)
endfunction()

# Fetch the sources from both directories and given files and gather them in
# one variable.
function(arcana_fetch_sources out)
  cmake_parse_arguments(_FUNC_ARG "" "" "" ${ARGN})
  arcana_fetch_files(src_files 
    ${_FUNC_ARG_UNPARSED_ARGUMENTS}
    EXTS ".cpp" ".c" ".cxx"
  )
  set(${out} ${src_files} PARENT_SCOPE)
endfunction()