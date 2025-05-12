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
# CMake display variables and macros
# =============================================================================
setup_display(4)

# =============================================================================
# CPP Autogen
# =============================================================================
dispSection("CPP header autogen")
arcana_autogen_rosd_cpp()

# =============================================================================
# Executables
# =============================================================================
dispSection("Configuring libraries")
dispStep("Registering library \"arcana_lifecycle\"")
ament_auto_add_library(arcana_lifecycle "src/lifecycle.cpp")
install(TARGETS arcana_lifecycle
  DESTINATION lib/${PROJECT_NAME})

# =============================================================================
# Resources
# =============================================================================
dispSection("Installing resources")
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/${PROJECT_NAME}
)

# =============================================================================
# Packaging everything
# =============================================================================
dispSection("Packaging everything (AMENT)")
ament_auto_package()