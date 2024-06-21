cmake_minimum_required(VERSION 3.8)
project(arcana_cmake NONE)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) 

find_package(ament_cmake REQUIRED)

include("cmake/compiling/cmake.cmake")
include("cmake/system/ros.cmake")
include("cmake/compiling/package.cmake")
include("cmake/watcher/watcher.cmake")

register_cmake(DIRS "cmake")
setup_project_watcher(_CUSTOM_ROOT "${CMAKE_CURRENT_LIST_DIR}/cmake" DISPLAY)

arcana_package()