cmake_minimum_required(VERSION 3.8)
project(arcana_tools CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -pedantic -Wno-return-type \
-std=c++${CMAKE_CXX_STANDARD} ${EXTRA_CC_FLAGS}")

find_package(ament_cmake_auto REQUIRED)
find_package(fmt REQUIRED)
ament_auto_find_build_dependencies()

include_directories("${CMAKE_CURRENT_SRC_DIR}/include")

# =============================================================================
# Executables
# =============================================================================
ament_auto_add_executable(tf_analyzer "src/tf_tools/tf_analyzer.cpp")
target_link_libraries(tf_analyzer fmt::fmt)
ament_auto_add_executable(tf_publisher "src/tf_tools/tf_publisher.cpp")

arcana_package()