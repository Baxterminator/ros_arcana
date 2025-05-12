cmake_minimum_required(VERSION 3.17)
project(arcana_python NONE)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) 

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# =============================================================================
# CMake display variables and macros
# =============================================================================
setup_display(3)

# =============================================================================
# Python Autogen
# =============================================================================
dispSection("Python module autogen")
arcana_autogen_rosd_py(DEST python/ros/distro.py)

# =============================================================================
# Install Python
# =============================================================================
dispSection("Python package configuring")
install(
    DIRECTORY python
    DESTINATION share/${PROJECT_NAME}
)
ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR python)

# =============================================================================
# Packaging
# =============================================================================
dispSection("Packaging everything (ARCANA)")
setup_project_watcher(DISPLAY)
arcana_package()