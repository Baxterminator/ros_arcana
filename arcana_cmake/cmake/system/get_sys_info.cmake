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

macro(fetch_system_info)
  cmake_parse_arguments(ARG "DISPLAY" "" "" ${ARGN})
  # Process
  check_raspberry_pi()

  # Display
  if(ARG_DISPLAY)
    message("System: ${CMAKE_SYSTEM} (cpu_arch=${CMAKE_SYSTEM_PROCESSOR})")
    message("Is a Raspberry System: ${IS_RASPBERRY}")
    message("CPU Model: ${CPU_MODEL}")
  endif()
endmacro()