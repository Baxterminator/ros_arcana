cmake_minimum_required(VERSION 3.8)
project(arcana_cpp CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -pedantic -Wno-return-type \
-std=c++${CMAKE_CXX_STANDARD} ${EXTRA_CC_FLAGS}")

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# =============================================================================
# Executables
# =============================================================================
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/${PROJECT_NAME}
)

ament_auto_package()