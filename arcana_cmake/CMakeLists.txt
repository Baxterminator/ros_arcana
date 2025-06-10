cmake_minimum_required(VERSION 3.17)
project(arcana_cmake NONE)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) 

find_package(ament_cmake REQUIRED)

include("cmake/utils/display.cmake")
include("cmake/compiling/cmake.cmake")
include("cmake/auto/gen_rosd.cmake")
include("cmake/compiling/package.cmake")
include("cmake/watcher/watcher.cmake")

# =============================================================================
# CMake display variables and macros
# =============================================================================
setup_display(3)

# =============================================================================
# Registering cmake files
# =============================================================================
dispSection("Configuring CMake files")
register_cmake(DIRS "cmake")

# =============================================================================
# Run autogen of cmake files
# =============================================================================
dispSection("CMake autogen configuring")
arcana_autogen_rosd_cmake()

# =============================================================================
# Packaging everything
# =============================================================================
dispSection("Packaging everything (ARCANA)")
# setup_project_watcher(_CUSTOM_ROOT "${CMAKE_CURRENT_LIST_DIR}/cmake" DISPLAY)
arcana_package()