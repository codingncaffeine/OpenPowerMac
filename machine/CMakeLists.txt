set(OPM_MACHINE_SRC
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
  src/snapshot.cpp
)

add_library(opmmachine STATIC ${OPM_MACHINE_SRC})
target_include_directories(opmmachine PUBLIC include)
target_link_libraries(opmmachine PUBLIC opmcore)
if(MSVC)
  target_compile_definitions(opmmachine PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()

# Profiling twin (see core/CMakeLists.txt). It exists only so that g4prof does
# not link two copies of the core with different definitions of the same
# inline functions, which is an ODR violation the linker will not report.
add_library(opmmachine_prof STATIC EXCLUDE_FROM_ALL ${OPM_MACHINE_SRC})
target_include_directories(opmmachine_prof PUBLIC include)
target_link_libraries(opmmachine_prof PUBLIC opmcore_prof)
if(MSVC)
  target_compile_definitions(opmmachine_prof PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()
