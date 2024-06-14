cmake_minimum_required(VERSION 3.8)
project(arcana_cmake NONE)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) 

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gmock REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

ament_package(
  CONFIG_EXTRAS "arcana_cmake_extras.cmake"
)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)