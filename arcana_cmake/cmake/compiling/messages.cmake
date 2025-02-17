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

macro(register_messages)
    cmake_parse_arguments(ARG "" "MSG_DIR;SRV_DIR;ACTION_DIR" "DEPS" ${ARGN})

    # Parse arguments
    set(msg_dir "msg")
    set(srv_dir "srv")
    set(action_dir "action")
    if (ARG_MSG_DIR)
        set(msg_dir ${ARG_MSG_DIR})
    endif()
    if (ARG_SRV_DIR)
        set(srv_dir ${ARG_srv_DIR})
    endif()
    if (ARG_ACTION_DIR)
        set(action_dir ${ARG_ACTION_DIR})
    endif()

    file(GLOB ${PROJEC_NAME}_msg_files RELATIVE "${CMAKE_CURRENT_LIST_DIR}"
    "${CMAKE_CURRENT_LIST_DIR}/${msg_dir}/*.msg"
    "${CMAKE_CURRENT_LIST_DIR}/${srv_dir}/*.srv"
    "${CMAKE_CURRENT_LIST_DIR}/${action_dir}/*.action"
    )
    set(${PROJECT_NAME}_msg_deps rosidl_default_runtime ${ARG_DEPS})
    if (${PROJEC_NAME}_msg_files)
        dispSection("Generating custom messages interfaces")
        foreach(msg ${${PROJEC_NAME}_msg_files})
            message("\t- Found message \"${msg}\"")
        endforeach()
        rosidl_generate_interfaces("${PROJECT_NAME}" ${${PROJEC_NAME}_msg_files} DEPENDENCIES ${${PROJECT_NAME}_msg_deps})
        rosidl_get_typesupport_target(${PROJECT_NAME}_CUSTOM_MSGS_LIB "${PROJECT_NAME}"  "rosidl_typesupport_cpp")
    else()
        dispSection("No messages to generate interface for")
    endif()
endmacro()