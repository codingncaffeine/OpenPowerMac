add_library(opmcore STATIC
  src/isa_table.cpp
  src/decode.cpp
  src/disasm.cpp
  src/cpu.cpp
  src/exec_int.cpp
  src/exceptions.cpp
  src/mmu.cpp
  src/softfp.cpp
  src/fpu.cpp
  src/altivec.cpp
)

target_include_directories(opmcore PUBLIC include)
