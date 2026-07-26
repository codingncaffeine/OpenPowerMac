add_library(opmmachine STATIC
  src/ati.cpp
  src/gossamer.cpp
  src/macio.cpp
  src/cuda.cpp
  src/pci.cpp
  src/pmu.cpp
  src/sawtooth.cpp
)

target_include_directories(opmmachine PUBLIC include)
target_link_libraries(opmmachine PUBLIC opmcore)
