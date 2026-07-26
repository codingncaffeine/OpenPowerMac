cmake_minimum_required(VERSION 3.21)
project(OpenPowerMac CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(OPM_BUILD_TESTS "Build unit tests" ON)

if(MSVC)
  add_compile_options(/W4 /WX /permissive- /utf-8)
else()
  add_compile_options(-Wall -Wextra -Werror)
endif()

add_subdirectory(core)
add_subdirectory(machine)
add_subdirectory(tools/ppcrun)
add_subdirectory(tools/sstgen)
add_subdirectory(tools/macrun)
add_subdirectory(tools/g4run)

if(OPM_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
