function(set_target_warning_compile_options TARGET)

  if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")

    set(TARGET_FLAGS -Wall -Wextra -Werror -pedantic -Weverything)
    set(WARNINGS_THAT_SHOULD_BE_IGNORED
        c++98-compat                    # Code base should be modern
        c++98-compat-pedantic           # Code base should be modern
        padded                          # It's not worth the effort in our domain (desktop PC with tons of memory)
        return-std-move-in-c++11        # Applies to old compilers, we aim for newer C++17 compilers
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

  else()
    message(FATAL_ERROR "Unhandled compiler vendor ${CMAKE_CXX_COMPILER_ID}")
  endif()

endfunction()
