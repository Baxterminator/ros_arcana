macro(fetch_system_info)
  cmake_parse_arguments(ARG "DISPLAY" "" "" ${ARGN})
  # Process
  check_raspberry_pi()

  # Display
  if(ARG_DISPLAY)
    message("System: ${CMAKE_SYSTEM} (cpu_arch=${CMAKE_SYSTEM_PROCESSOR})")
    message("Is a Raspberry System: ${IS_RASPBERRY}")
    message("CPU Model: ${CPU_MODEL}")
  endif()
endmacro()