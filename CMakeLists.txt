cmake_minimum_required(VERSION 3.21)
project(OpenPowerMac CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# opmcapi is SHARED and links the static core/machine libs; ELF needs those built PIC
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(OPM_BUILD_TESTS "Build unit tests" ON)

# ⭐ CROSS-TU INLINING (Release). This machine's hot path crosses translation
# units on EVERY emulated instruction: runSteps (core/src/cpu.cpp) calls the
# handlers (exec_int.cpp, fpu.cpp), those call readV/writeV (mmu.cpp), which
# call memRead/memWrite (cache.cpp), and every floating-point operation calls
# into softfp.cpp and back. Compiled one TU at a time, each of those is an
# opaque call across an ABI boundary the optimizer cannot see through — and
# per-call cost is exactly where the --jit-tsc split found this machine's
# remaining time (fallbacks 45.9% of block time at ~110 cycles a call, memory
# 24.9% at ~57).
#
# It is a pure optimizer setting: no source change, no semantic change (FP
# stays /fp:precise), so its claim is "the same program, faster" — and that
# claim is settled the way every other one here is, by whole-machine
# fingerprints across an in-game window plus the standing cold-boot gates,
# never by trusting the label. OPM_LTO=OFF is the control.
option(OPM_LTO "Cross-TU optimization in Release builds" ON)
if(OPM_LTO)
  include(CheckIPOSupported)
  check_ipo_supported(RESULT opm_ipo_ok OUTPUT opm_ipo_msg)
  if(opm_ipo_ok)
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE)
    message(STATUS "OpenPowerMac: cross-TU optimization ON for Release")
  else()
    message(STATUS "OpenPowerMac: cross-TU optimization unavailable "
                   "(${opm_ipo_msg})")
  endif()
endif()

if(MSVC)
  add_compile_options(/W4 /WX /permissive- /utf-8)
else()
  add_compile_options(-Wall -Wextra -Werror)
endif()

add_subdirectory(core)
add_subdirectory(machine)
add_subdirectory(capi)
add_subdirectory(tools/ppcrun)
add_subdirectory(tools/sstgen)
add_subdirectory(tools/macrun)
add_subdirectory(tools/g4run)
add_subdirectory(tools/mkhfs)
add_subdirectory(tools/ppcdis)

if(OPM_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
