add_library(opmcapi SHARED
  src/capi.cpp
)

target_include_directories(opmcapi PUBLIC include)
target_link_libraries(opmcapi PRIVATE opmmachine opmcore)
if(MSVC)
  target_compile_definitions(opmcapi PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()

# Only the C ABI leaves the library. Everything else stays inside it, the C++
# runtime's own instantiations included: a library loaded beside this one in the
# app's process can then neither take over one of its calls nor have one of its
# own taken. (Windows exports by name already: OPM_API.)
if(UNIX AND NOT APPLE)
  target_link_options(opmcapi PRIVATE
    "LINKER:--version-script=${CMAKE_CURRENT_SOURCE_DIR}/exports.map")
  set_property(TARGET opmcapi APPEND PROPERTY LINK_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/exports.map")
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

# The Linux front end (gui/, Avalonia) picks the library up from gui/native/
# (gitignored), the same way.
if(UNIX AND NOT APPLE)
  add_custom_command(TARGET opmcapi POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:opmcapi>
            ${CMAKE_SOURCE_DIR}/gui/native/libopmcapi.so)
endif()
