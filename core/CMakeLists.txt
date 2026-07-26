add_library(opmcore STATIC
  src/isa_table.cpp
  src/decode.cpp
  src/disasm.cpp
  src/cpu.cpp
)

target_include_directories(opmcore PUBLIC include)
