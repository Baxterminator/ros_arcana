cmake_minimum_required(VERSION 3.8)
project(arcana_python NONE)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) 

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

install(
    DIRECTORY python
    DESTINATION share/${PROJECT_NAME}
)
ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR python)

setup_project_watcher(DISPLAY)
arcana_package()