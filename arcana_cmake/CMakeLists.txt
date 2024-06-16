cmake_minimum_required(VERSION 3.8)
project(arcana_cmake NONE)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) 

find_package(ament_cmake REQUIRED)

include("cmake/compiling/cmake.cmake")

ament_add_cmake(DIRS "cmake" "hello")
