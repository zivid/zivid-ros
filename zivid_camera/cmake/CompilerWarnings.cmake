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
    # To be able to discover issues in our code as new compilers are released,
    # we enable -Werror and -Weverything and suppress warnings when necessary.
    # However, this means that warnings that are introduced in one clang version
    # may not be available in another. Hence, we only enable -Weverything for
    # version greater or equal to the newest version we test in CI. This also
    # ensures that once we introduce a newer compiler version, CI will fail
    # unless we address or suppress any new warnings from that version.

    set(SUPPORTED_CLANG_WARNINGS_VERSION 22)
    if(${CMAKE_CXX_COMPILER_VERSION} GREATER_EQUAL ${SUPPORTED_CLANG_WARNINGS_VERSION})

      set(TARGET_FLAGS -Wall -Wextra -Werror -pedantic -Weverything)
      set(WARNINGS_THAT_SHOULD_BE_IGNORED
        c++98-compat                    # Code base should be modern
        c++98-compat-pedantic           # Code base should be modern
        padded                          # It's not worth the effort in our domain (desktop PC with tons of memory)
        weak-vtables                    # The vtable must be duplicated in multiple translation units. Small
                                        # problem, maybe even linker will resolve this. Must add boilerplate to
                                        # fix, not worth it
        covered-switch-default          # We don't want this warning, because we want the default labels for safety.
        unsafe-buffer-usage-in-libc-call# ROS macros trigger such warnings
        missing-noreturn                # Triggers in visitor-style code with a throwing branch. Adding the label is a
                                        # C++23 extension.
        c2y-extensions                  # Triggers with some ROS macros using __COUNTER__ builtin.
      )

      foreach(WARNING ${WARNINGS_THAT_SHOULD_BE_IGNORED})
        list(APPEND TARGET_FLAGS -Wno-${WARNING})
      endforeach()

    else()

      # Enable some warnings, but not -Weverything, for the reason explained above.
      set(TARGET_FLAGS -Wall -Wextra -Werror)

    endif()

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
