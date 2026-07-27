add_executable(opm_unit
  unit/main.cpp
  unit/test_bits.cpp
  unit/test_decode.cpp
  unit/test_mmu.cpp
  unit/test_chip.cpp
  unit/test_ati.cpp
  unit/test_macio.cpp
  unit/test_snapshot.cpp
)
target_link_libraries(opm_unit PRIVATE opmcore opmmachine)
target_include_directories(opm_unit SYSTEM PRIVATE vendor)
if(MSVC)
  # Vendored doctest is third-party; exempt it from new-compiler warnings.
  target_compile_options(opm_unit PRIVATE /wd5285)
  # Same C-runtime posture as the machine library the tests exercise, which
  # opens image files with plain fopen.
  target_compile_definitions(opm_unit PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()

add_test(NAME unit COMMAND opm_unit)
add_test(NAME kat COMMAND ppcrun --kat ${CMAKE_CURRENT_SOURCE_DIR}/kat)

add_test(NAME sst_gen COMMAND sstgen --out ${CMAKE_BINARY_DIR}/sst --count 100)
set_tests_properties(sst_gen PROPERTIES FIXTURES_SETUP sstdata)
add_test(NAME sst COMMAND ppcrun --sst ${CMAKE_BINARY_DIR}/sst)
set_tests_properties(sst PROPERTIES FIXTURES_REQUIRED sstdata)
