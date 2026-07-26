add_executable(opm_unit
  unit/main.cpp
  unit/test_bits.cpp
  unit/test_decode.cpp
)
target_link_libraries(opm_unit PRIVATE opmcore)
target_include_directories(opm_unit SYSTEM PRIVATE vendor)
if(MSVC)
  # Vendored doctest is third-party; exempt it from new-compiler warnings.
  target_compile_options(opm_unit PRIVATE /wd5285)
endif()

add_test(NAME unit COMMAND opm_unit)
add_test(NAME kat COMMAND ppcrun --kat ${CMAKE_CURRENT_SOURCE_DIR}/kat)

add_test(NAME sst_gen COMMAND sstgen --out ${CMAKE_BINARY_DIR}/sst --count 100)
set_tests_properties(sst_gen PROPERTIES FIXTURES_SETUP sstdata)
add_test(NAME sst COMMAND ppcrun --sst ${CMAKE_BINARY_DIR}/sst)
set_tests_properties(sst PROPERTIES FIXTURES_REQUIRED sstdata)
