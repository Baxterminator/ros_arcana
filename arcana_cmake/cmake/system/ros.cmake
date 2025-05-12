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

include("${CMAKE_CURRENT_LIST_DIR}/../rosd_autogen.cmake")

function(get_ros_version)
  cmake_parse_arguments(ARG "" "OUT" "" ${ARGN})
  if(DEFINED ENV{ROS_DISTRO})
    str_to_rosd($ENV{ROS_DISTRO} _semantic)
    set(${ARG_OUT} ${_semantic})
  else()
    message(SEND_ERROR "Could not fetch ROS_DISTRO environement variable.")
    return()
  endif()
endfunction()