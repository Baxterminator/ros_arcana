cmake_minimum_required(VERSION 3.8)
project(arcana_gazebo NONE)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) 

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Install gazebo models
install(
  DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

# Run scripts
set(ARUCO_FORMATS "4;5;6;7;original")
foreach(format ${ARUCO_FORMATS})
  install(
    CODE "message(\"[Arcana ArUco] Creating ArUco dictionary for format ${format}x${format}\")\nexec_program(\"python3 ${CMAKE_CURRENT_LIST_DIR}/scripts/aruco_extractor.py ${format} -o ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/data/aruco_${format}x${format}_1000.yaml\")"
  )  
endforeach()

setup_project_watcher(DISPLAY)
arcana_package()