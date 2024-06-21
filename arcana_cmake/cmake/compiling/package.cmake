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

# Fixed version of the ament_auto_package() macro from ament_auto to allow passing CONFIG_EXTRAS to ament_package
# that can be used until fixes are implemented.
macro(_fixed_ament_auto)
  # Copyright 2014 Open Source Robotics Foundation, Inc.
  #
  # Licensed under the Apache License, Version 2.0 (the "License");
  # you may not use this file except in compliance with the License.
  # You may obtain a copy of the License at
  #
  #     http://www.apache.org/licenses/LICENSE-2.0
  #
  # Unless required by applicable law or agreed to in writing, software
  # distributed under the License is distributed on an "AS IS" BASIS,
  # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  # See the License for the specific language governing permissions and
  # limitations under the License.

  #
  # Export information, install files and targets, execute the
  # extension point ``ament_auto_package`` and invoke
  # ``ament_package()``.
  #
  # :param INSTALL_TO_PATH: if set, install executables to `bin` so that
  #   they are available on the `PATH`.
  #   By default they are being installed into `lib/${PROJECT_NAME}`.
  #   It is currently not possible to install some executable into `bin`
  #   and some into `lib/${PROJECT_NAME}`.
  #   Libraries are not affected by this option.
  #   They are always installed into `lib` and `dll`s into `bin`.
  # :type INSTALL_TO_PATH: option
  # :param INSTALL_TO_SHARE: a list of directories to be installed to the
  #   package's share directory
  # :type INSTALL_TO_SHARE: list of strings
  # :param ARGN: any other arguments are passed through to ament_package()
  # :type ARGN: list of strings
  #
  # Export all found build dependencies which are also run
  # dependencies.
  # If the package has an include directory install all recursively
  # found header files (ending in h, hh, hpp, hxx) and export the
  # include directory.
  # Export and install all library targets and install all executable
  # targets.

  cmake_parse_arguments(ARG_AMENT_AUTO_PACKAGE "INSTALL_TO_PATH" "" "INSTALL_TO_SHARE" ${ARGN})
  # passing all unparsed arguments to ament_package()

  # export all found build dependencies which are also run dependencies
  set(_run_depends
    ${${PROJECT_NAME}_BUILD_EXPORT_DEPENDS}
    ${${PROJECT_NAME}_BUILDTOOL_EXPORT_DEPENDS}
    ${${PROJECT_NAME}_EXEC_DEPENDS})
  foreach(_dep
      ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS}
      ${${PROJECT_NAME}_FOUND_BUILDTOOL_DEPENDS})
    if(_dep IN_LIST _run_depends)
      ament_export_dependencies("${_dep}")
    endif()
  endforeach()

  # export and install include directory of this package if it has one
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include")
    ament_export_include_directories("include")
    install(DIRECTORY include/ DESTINATION include)
  endif()

  # export and install all libraries
  if(NOT ${PROJECT_NAME}_LIBRARIES STREQUAL "")
    set(without_interfaces "")
    foreach(library_name ${${PROJECT_NAME}_LIBRARIES})
      get_target_property(library_type ${library_name} TYPE)
      if(NOT "${library_type}" STREQUAL "INTERFACE_LIBRARY")
        list(APPEND without_interfaces ${library_name})
      endif()
    endforeach()

    ament_export_libraries(${without_interfaces})
    install(
      TARGETS ${without_interfaces}
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
    )
  endif()

  # install all executables
  if(NOT ${PROJECT_NAME}_EXECUTABLES STREQUAL "")
    if(ARG_AMENT_AUTO_PACKAGE_INSTALL_TO_PATH)
      set(_destination "bin")
    else()
      set(_destination "lib/${PROJECT_NAME}")
    endif()
    install(
      TARGETS ${${PROJECT_NAME}_EXECUTABLES}
      DESTINATION ${_destination}
    )
  endif()

  # install directories to share
  foreach(_dir ${ARG_AMENT_AUTO_PACKAGE_INSTALL_TO_SHARE})
    install(
      DIRECTORY "${_dir}"
      DESTINATION "share/${PROJECT_NAME}"
    )
  endforeach()

  ament_execute_extensions(ament_auto_package)

  ament_package(${ARG_AMENT_AUTO_PACKAGE_UNPARSED_ARGUMENTS})
endmacro()

# -----------------------------------------------------------------------------

# Wrapper for the packing function that include all functionnality 
# that are needed by the arcana_cmake package
macro(arcana_package)
  cmake_parse_arguments(ARG "" "" "" ${ARGN})

  make_extras_file()
  get_ros_version()

  # If ROS distro is before jazzy, then patch likely hasn't been made, so rely on a fixed function.
  if (${ROS_SUBVERSION} VERSION_LESS ${JAZZY_SUBVERSION})
    _fixed_ament_auto(
      CONFIG_EXTRAS ${ARCANA_${PROJECT_NAME}_EXTRA_CMAKE_PATH}
      ${ARG_UNPARSED_ARGUMENTS}
    )
  else()
    ament_auto_package(
      CONFIG_EXTRAS ${ARCANA_${PROJECT_NAME}_EXTRA_CMAKE_PATH}
      ${ARG_UNPARSED_ARGUMENTS}
    )
  endif()
endmacro()