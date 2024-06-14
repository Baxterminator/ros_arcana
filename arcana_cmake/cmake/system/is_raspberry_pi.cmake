# Copyright (C) 2024-2024 by Meltwin
# Authors:
#   Geoffrey Côte: geoffrey.cote@centraliens-nantes.org

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

function(check_raspberry_pi)
  exec_program("cat /proc/cpuinfo | grep model -i -m 2" OUTPUT_VARIABLE cpu_output)
  string(FIND ${cpu_output} "Raspberry" has_raspberry)

  # Found a raspberry, fetching data
  if(NOT ${has_raspberry} EQUAL -1)
    set(IS_RASPBERRY YES PARENT_SCOPE)

    # Fetch raspberry pi model
    string(REGEX MATCH "(Raspberry.+$)" model_name ${cpu_output})
    set(RASPBERRY_MODEL ${model_name} PARENT_SCOPE)

  # Not a raspberry, still fetch the data of the CPU
  else()
    set(IS_RASPBERRY NO PARENT_SCOPE)

    # Fetch CPU Model
    string(REGEX MATCH "([mM]odel [nN]ame)" has_model_name ${cpu_output})
    if(has_model_name)
      string(REGEX MATCH "[mM]odel [nN]ame.+: .+" model_name ${cpu_output})
      string(REGEX REPLACE "[mM]odel [nN]ame.+: " "" model_name ${model_name})
    else()
      string(REGEX MATCH "[mM]odel.+:.+\n" model_name ${cpu_output})
      string(REGEX REPLACE "[mM]odel.+: " "" model_name ${model_name})
    endif()
    string(STRIP ${model_name} model_name)
  endif()

  set(CPU_MODEL ${model_name} PARENT_SCOPE)
endfunction()