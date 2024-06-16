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

find_package(ament_cmake QUIET REQUIRED)

# Package, create extra files and install cmake functions
macro(ament_add_cmake)
  cmake_parse_arguments(ARG "" "" "FILES;DIRS" ${ARGN})

  if (NOT ARG_FILES AND NOT ARG_DIRS)
    message(SEND_ERROR "No file or directory provided for adding cmake files to the ament stack")
    return()
  endif()

  message("Installing cmake files with\n   -> ARG_FILES: ${ARG_FILES}\n   -> ARG_DIRS: ${ARG_DIRS}")

  # Install directories
  if (ARG_DIRS)
    foreach(dir ${ARG_DIRS})
      # Check if directory exist and get relative directory path
      if (IS_ABSOLUTE ${dir})
        if (NOT EXISTS ${dir})
          message(WARNING "Directory ${dir} does not exist!")
          continue()
        endif()
        file(RELATIVE_PATH rel_dir ${CMAKE_CURRENT_LIST_DIR} ${dir})
      else()
        if (NOT EXISTS "${CMAKE_CURRENT_LIST_DIR}/${dir}")
          message(WARNING "Directory ${dir} does not exist!")
          continue()
        endif()
        set(rel_dir ${dir})
      endif()

      # Copy everything inside of these directories
      file(GLOB_RECURSE files LIST_DIRECTORIES false RELATIVE "${CMAKE_CURRENT_LIST_DIR}/${rel_dir}" "${rel_dir}/**/*.cmake" "${rel_dir}/*.cmake")
      foreach(file ${files})
        get_filename_component(file_dir ${file} DIRECTORY)
        list(APPEND found_files ${file})
        install(
          FILES "${rel_dir}/${file}"
          DESTINATION share/${PROJECT_NAME}/cmake/${file_dir}
        ) 
      endforeach()
    endforeach()
  endif()

  # Install files
  if (ARG_FILES)
    foreach(file ${ARG_FILES})
      # Check if file exist
      if (IS_ABSOLUTE ${file})
        if (NOT EXISTS ${file})
          message(WARNING "File ${file} does not exist!")
          continue()
        endif()
      else()
        if (NOT EXISTS "${CMAKE_CURRENT_LIST_DIR}/${file}")
          message(WARNING "File ${file} does not exist!")
          continue()
        endif()
      endif()

      # Install file 
      list(APPEND found_files ${file})
      install(
        FILES "${file}"
        DESTINATION share/${PROJECT_NAME}/cmake/
      )
    endforeach()
  endif()

  # Make extras file
  set(extras_file_path "${CMAKE_CURRENT_BINARY_DIR}/autogen_${PROJECT_NAME}_extras.cmake")
  set(extras_content "find_package(ament_cmake QUIET REQUIRED)")
  foreach(file ${found_files})
    set(extras_content "${extras_content}\ninclude(\${${PROJECT_NAME}_DIR}/${file})")
  endforeach()
  file(WRITE ${extras_file_path} ${extras_content})  

  # Add it to ament
  ament_package(
    CONFIG_EXTRAS ${extras_file_path}
  )

endmacro()