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

set(ARCANA_XACRO_PREFIX "[Arcana Xacro] ")

# Run Xacro files and install them in the share folder
function(install_xacro_files)
  cmake_parse_arguments(ARG "" "" "FILES" ${ARGN})

  set(install_dir "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}")
  message("Generating xacro files compiling target:\n   -> Output will be in ${install_dir}")

  foreach(file ${ARG_FILES})
    # Process file path to get relative path
    if (IS_ABSOLUTE ${file})
      foreach_file_exist(${file})
      file(RELATIVE_PATH rel_path ${CMAKE_CURRENT_LIST_DIR} ${file})
    else()
      foreach_file_exist("${CMAKE_CURRENT_LIST_DIR}/${file}")
      set(rel_path ${file})
    endif()

    # Replace the extension
    string(REGEX REPLACE ".xacro" "" out_path ${rel_path})

    set(xacro_cmd "exec_program(\"xacro ${CMAKE_CURRENT_LIST_DIR}/${rel_path} -o ${install_dir}/${out_path}\" OUTPUT_VARIABLE out)\n")
    string(APPEND xacro_cmd "if(\${out})\nmessage(\"${ARCANA_XACRO_PREFIX}Error when compiling xacro file ${file}:\n\${out}\")\nelse()\n")
    string(APPEND xacro_cmd "message(\"${ARCANA_XACRO_PREFIX}Compiled xacro ${file}\n \${out}\")\n")
    string(APPEND xacro_cmd "file(READ ${install_dir}/${out_path} file_content)\n")
    string(APPEND xacro_cmd "string(REGEX REPLACE \"<!--[^<>]*-->\n\" \"\" file_content_out \${file_content})\n")
    string(APPEND xacro_cmd "file(WRITE ${install_dir}/${out_path} \${file_content_out})\n")
    string(APPEND xacro_cmd "endif()")

    # Add command to compile it
    message("   -> Configuring ${rel_path}")
    install(CODE ${xacro_cmd})
  endforeach()
endfunction()