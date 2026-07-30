add_executable(g4run main.cpp)
target_link_libraries(g4run PRIVATE opmmachine)
if(MSVC)
  target_compile_definitions(g4run PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()

# The same program with opm/prof.hpp's markers compiled in. Built on demand
# (EXCLUDE_FROM_ALL) because it is a measuring instrument, not an artifact:
#   cmake --build build --config Release --target g4prof
add_executable(g4prof EXCLUDE_FROM_ALL main.cpp)
target_link_libraries(g4prof PRIVATE opmmachine_prof)
if(MSVC)
  target_compile_definitions(g4prof PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()
