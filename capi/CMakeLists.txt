add_library(opmcapi SHARED
  src/capi.cpp
)

target_include_directories(opmcapi PUBLIC include)
target_link_libraries(opmcapi PRIVATE opmmachine opmcore)
if(MSVC)
  target_compile_definitions(opmcapi PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()

# The WPF shell picks the DLL up from shell/native/ (gitignored).
#
# MSVC ONLY, and the guard is load-bearing: MinGW is also WIN32, so a
# `cmake --build build-gcc` used as a second-compiler check would copy its own
# libopmcapi.dll over this one. The shell would then ship a gcc-built DLL with
# no libstdc++/libgcc_s/libwinpthread beside it and stop booting, while every
# native build and every test still passed. That happened.
if(WIN32 AND MSVC)
  add_custom_command(TARGET opmcapi POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:opmcapi>
            ${CMAKE_SOURCE_DIR}/shell/native/opmcapi.dll)
endif()
