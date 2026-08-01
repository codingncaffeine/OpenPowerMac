add_executable(mkhfs main.cpp)
target_link_libraries(mkhfs PRIVATE opmmachine)
if(MSVC)
  target_compile_definitions(mkhfs PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()
if(MINGW)
  # wmain needs the unicode startup shim on MinGW; MSVC finds it by itself.
  target_link_options(mkhfs PRIVATE -municode)
endif()
