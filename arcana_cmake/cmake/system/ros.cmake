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

set(ARDENT_SUBVERSION "2.1")
set(BOUNCY_SUBVERSION "2.2")
set(CRYSTAL_SUBVERSION "2.3")
set(DASHING_SUBVERSION "2.4")
set(ELOQUENT_SUBVERSION "2.5")
set(FOXY_SUBVERSION "2.6")
set(GALACTIC_SUBVERSION "2.7")
set(HUMBLE_SUBVERSION "2.8")
set(IRON_SUBVERSION "2.9")
set(JAZZY_SUBVERSION "2.10")

# Fetch the ROS version from the environement and set a major.minor format
# Correspondance table:
#   - 2.1 -> ardent
#   - 2.2 -> bouncy
#   - 2.3 -> crystal
#   - 2.4 -> dashing
#   - 2.5 -> eloquent
#   - 2.6 -> foxy
#   - 2.7 -> galactic
#   - 2.8 -> humble
#   - 2.9 -> iron
#   - 2.10 -> jazzy
#     ....
macro(get_ros_version)
  cmake_parse_arguments(ARG "" "OUT" "" ${ARGN})

  if(DEFINED ENV{ROS_DISTRO})
    set(_subversion "$ENV{ROS_DISTRO}")

    if (${_subversion} STREQUAL "ardent")
      set(ROS_SUBVERSION ${ARDENT_SUBVERSION})
    elseif(${_subversion} STREQUAL "bouncy")
      set(ROS_SUBVERSION ${BOUNCY_SUBVERSION})
    elseif(${_subversion} STREQUAL "crystal")
      set(ROS_SUBVERSION ${CRYSTAL_SUBVERSION})
    elseif(${_subversion} STREQUAL "dashing")
      set(ROS_SUBVERSION ${DASHING_SUBVERSION})
    elseif(${_subversion} STREQUAL "eloquent")
      set(ROS_SUBVERSION ${ELOQUENT_SUBVERSION})
    elseif(${_subversion} STREQUAL "foxy")
      set(ROS_SUBVERSION ${FOXY_SUBVERSION})
    elseif(${_subversion} STREQUAL "galactic")
      set(ROS_SUBVERSION ${GALACTIC_SUBVERSION})
    elseif(${_subversion} STREQUAL "humble")
      set(ROS_SUBVERSION ${HUMBLE_SUBVERSION})
    elseif(${_subversion} STREQUAL "iron")
      set(ROS_SUBVERSION ${IRON_SUBVERSION})
    elseif(${_subversion} STREQUAL "jazzy")
      set(ROS_SUBVERSION ${JAZZY_SUBVERSION})
    else()
      set(ROS_SUBVERSION "1.0")
      message(WARNING "ROS_DISTRO does not correspond to any of the registered ROS distro: '${_subversion}'")
    endif()

  else()
    message(SEND_ERROR "Could not fetch ROS_DISTRO environement variable.")
    return()
  endif()

  if (ARG_OUT)
    set(${ARG_OUT} ${ROS_SUBVERSION})
  endif()
endmacro()