# =============================================================================
#                        ROS2 installation utils methods                       
# Auto-generated file that contains the ROS distro mappings
# 
# Copyright (c) Meltwin 2024 - 2025
# Author: Geoffrey Côte
# Part of the ros_arcana package
# 
# The MIT License (MIT)  https://mit-license.org/
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the “Software”), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell co
# pies of the Software, and to permit persons to whom the Software is furnished
# to do so, subject to the following conditions:
# The above copyright notice and this permission notice shall be included in al
# l copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IM
# PLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNES
# S FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
# OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WH
# ETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
# CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
# =============================================================================


set(ROSD_UNKNOWN "0.0")
set(ROSD_ARDENT "2.0")
set(ROSD_BOUNCY "2.1")
set(ROSD_CRYSTAL "2.2")
set(ROSD_DASHING "2.3")
set(ROSD_ELOQUENT "2.4")
set(ROSD_FOXY "2.5")
set(ROSD_GALACTIC "2.6")
set(ROSD_HUMBLE "2.7")
set(ROSD_IRON "2.8")
set(ROSD_JAZZY "2.9")

# Return the equivalent version of the ROS distro.
function (str_to_rosd v out)
  if (${v} STREQUAL "ardent")
    set(${out} ${ROSD_ARDENT} PARENT_SCOPE)
    return()
  elseif (${v} STREQUAL "bouncy")
    set(${out} ${ROSD_BOUNCY} PARENT_SCOPE)
    return()
  elseif (${v} STREQUAL "crystal")
    set(${out} ${ROSD_CRYSTAL} PARENT_SCOPE)
    return()
  elseif (${v} STREQUAL "dashing")
    set(${out} ${ROSD_DASHING} PARENT_SCOPE)
    return()
  elseif (${v} STREQUAL "eloquent")
    set(${out} ${ROSD_ELOQUENT} PARENT_SCOPE)
    return()
  elseif (${v} STREQUAL "foxy")
    set(${out} ${ROSD_FOXY} PARENT_SCOPE)
    return()
  elseif (${v} STREQUAL "galactic")
    set(${out} ${ROSD_GALACTIC} PARENT_SCOPE)
    return()
  elseif (${v} STREQUAL "humble")
    set(${out} ${ROSD_HUMBLE} PARENT_SCOPE)
    return()
  elseif (${v} STREQUAL "iron")
    set(${out} ${ROSD_IRON} PARENT_SCOPE)
    return()
  elseif (${v} STREQUAL "jazzy")
    set(${out} ${ROSD_JAZZY} PARENT_SCOPE)
    return()
  else()
    set(${out} ${ROSD_UNKNOWN} PARENT_SCOPE)
    message(WARNING "${v} does not correspond to any of the registered ROS distro")
  endif()
endfunction()

# Return a stringified version of the ROS distro code.
function (rosd_to_str v out)
  if (${v} VERSION_EQUAL "2.0")
    set(${out} "ardent" PARENT_SCOPE)
    return()
  elseif (${v} VERSION_EQUAL "2.1")
    set(${out} "bouncy" PARENT_SCOPE)
    return()
  elseif (${v} VERSION_EQUAL "2.2")
    set(${out} "crystal" PARENT_SCOPE)
    return()
  elseif (${v} VERSION_EQUAL "2.3")
    set(${out} "dashing" PARENT_SCOPE)
    return()
  elseif (${v} VERSION_EQUAL "2.4")
    set(${out} "eloquent" PARENT_SCOPE)
    return()
  elseif (${v} VERSION_EQUAL "2.5")
    set(${out} "foxy" PARENT_SCOPE)
    return()
  elseif (${v} VERSION_EQUAL "2.6")
    set(${out} "galactic" PARENT_SCOPE)
    return()
  elseif (${v} VERSION_EQUAL "2.7")
    set(${out} "humble" PARENT_SCOPE)
    return()
  elseif (${v} VERSION_EQUAL "2.8")
    set(${out} "iron" PARENT_SCOPE)
    return()
  elseif (${v} VERSION_EQUAL "2.9")
    set(${out} "jazzy" PARENT_SCOPE)
    return()
  else()
    set(${out} "unknown" PARENT_SCOPE)
    message(WARNING "${v} does not correspond to any version code of the registered ROS distro")
  endif()
endfunction()

