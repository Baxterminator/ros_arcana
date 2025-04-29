# Copyright (C) 2024-2025 by Meltwin
# Authors:
#   Geoffrey Côte: github@meltwin.fr

# The MIT License (MIT)  https://mit-license.org/

# Permission is hereby granted, free of charge, to any person obtaining a copy of this software
# and associated documentation files (the “Software”), to deal in the Software without
# restriction, including without limitation the rights to use, copy, modify, merge, publish,
# distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all copies
# or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
# INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
# PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
# FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
# ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

if(NOT WIN32)
  string(ASCII 27 Esc)
  set(TERM_ColourReset "${Esc}[m")
  set(TERM_ColourBold  "${Esc}[1m")
  set(TERM_Red         "${Esc}[31m")
  set(TERM_Green       "${Esc}[32m")
  set(TERM_Yellow      "${Esc}[33m")
  set(TERM_Blue        "${Esc}[34m")
  set(TERM_Magenta     "${Esc}[35m")
  set(TERM_Cyan        "${Esc}[36m")
  set(TERM_White       "${Esc}[37m")
  set(TERM_Black       "${Esc}[40m")
  set(TERM_BoldRed     "${Esc}[1;31m")
  set(TERM_BoldGreen   "${Esc}[1;32m")
  set(TERM_BoldYellow  "${Esc}[1;33m")
  set(TERM_BoldBlue    "${Esc}[1;34m")
  set(TERM_BoldMagenta "${Esc}[1;35m")
  set(TERM_BoldCyan    "${Esc}[1;36m")
  set(TERM_BoldWhite   "${Esc}[1;37m")
endif()

macro(setup_display n_steps)
    set(N_STEPS ${n_steps})
    set(STEP 1)
endmacro()

macro(incStep)
  cmake_parse_arguments(_MACRO_ARG "PARENT" "" "" ${ARGN})
  math(EXPR STEP "${STEP}+1")
  if (_MACRO_ARG_PARENT)
    set(STEP ${STEP} PARENT_SCOPE)
  endif()
endmacro()

macro(dispSection text)
  message("[${TERM_Yellow}${STEP}/${N_STEPS}${TERM_ColourReset}] ${TERM_Cyan}${text}${TERM_ColourReset}")
  incStep(${ARGN})
endmacro()

function(dispStep text)
  message(" - ${TERM_Green}${text}${TERM_ColourReset}")
endfunction()

function(dispLine text)
  message(STATUS "${TERM_Black}    ¤ ${text}${TERM_ColourReset}")
endfunction()