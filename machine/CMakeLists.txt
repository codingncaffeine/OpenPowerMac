add_library(opmmachine STATIC
  src/ati.cpp
  src/gossamer.cpp
  src/macio.cpp
  src/cuda.cpp
  src/pci.cpp
  src/pmu.cpp
  src/sawtooth.cpp
  src/ata.cpp
)

target_include_directories(opmmachine PUBLIC include)
target_link_libraries(opmmachine PUBLIC opmcore)
if(MSVC)
  target_compile_definitions(opmmachine PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()
