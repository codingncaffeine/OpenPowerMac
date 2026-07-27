add_library(opmmachine STATIC
  src/ati.cpp
  src/gossamer.cpp
  src/macio.cpp
  src/cuda.cpp
  src/pci.cpp
  src/pmu.cpp
  src/sawtooth.cpp
  src/ata.cpp
  src/openpic.cpp
  src/ohci.cpp
  src/r128.cpp
  src/dbdma.cpp
)

target_include_directories(opmmachine PUBLIC include)
target_link_libraries(opmmachine PUBLIC opmcore)
if(MSVC)
  target_compile_definitions(opmmachine PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()
