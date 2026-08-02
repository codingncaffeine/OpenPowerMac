set(OPM_CORE_SRC
  src/isa_table.cpp
  src/decode.cpp
  src/disasm.cpp
  src/cpu.cpp
  src/exec_int.cpp
  src/exceptions.cpp
  src/mmu.cpp
  src/cache.cpp
  src/softfp.cpp
  src/fpu.cpp
  src/altivec.cpp
  src/prof.cpp
  src/jit.cpp
)

find_package(Threads REQUIRED)

add_library(opmcore STATIC ${OPM_CORE_SRC})
target_include_directories(opmcore PUBLIC include)
target_link_libraries(opmcore PUBLIC Threads::Threads)

# The same sources with the profiler's markers compiled in (see opm/prof.hpp).
# A second target rather than a runtime switch: the marker sits on the hottest
# path in the program, and a machine that pays to measure itself even when
# nobody is measuring carries a permanent tax. g4prof links this; everything
# that ships links opmcore.
add_library(opmcore_prof STATIC EXCLUDE_FROM_ALL ${OPM_CORE_SRC})
target_include_directories(opmcore_prof PUBLIC include)
target_link_libraries(opmcore_prof PUBLIC Threads::Threads)
target_compile_definitions(opmcore_prof PUBLIC OPM_PROFILE=1)
