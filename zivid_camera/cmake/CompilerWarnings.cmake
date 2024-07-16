# Copyright 2024 Zivid AS
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Zivid AS nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

function(set_target_warning_compile_options TARGET)

  if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")

    set(TARGET_FLAGS -Wall -Wextra -Werror -pedantic -Weverything)
    set(WARNINGS_THAT_SHOULD_BE_IGNORED
        c++98-compat                    # Code base should be modern
        c++98-compat-pedantic           # Code base should be modern
        padded                          # It's not worth the effort in our domain (desktop PC with tons of memory)
        weak-vtables                    # The vtable must be duplicated in multiple translation units. Small
                                        # problem, maybe even linker will resolve this. Must add boilerplate to
                                        # fix, not worth it
    )

    foreach(WARNING ${WARNINGS_THAT_SHOULD_BE_IGNORED})
      list(APPEND TARGET_FLAGS -Wno-${WARNING})
    endforeach()

    target_compile_options(${TARGET} PRIVATE ${TARGET_FLAGS})

  elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")

    set(TARGET_FLAGS -Wall -Wextra -Werror -pedantic)
    target_compile_options(${TARGET} PRIVATE ${TARGET_FLAGS})

  elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")

      set(TARGET_FLAGS /W4 /WX /permissive-)
      target_compile_options(${TARGET} PRIVATE ${TARGET_FLAGS})

  else()
    message(FATAL_ERROR "Unhandled compiler vendor ${CMAKE_CXX_COMPILER_ID}")
  endif()

endfunction()
