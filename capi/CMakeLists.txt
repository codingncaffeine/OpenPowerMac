add_library(opmcapi SHARED
  src/capi.cpp
)

target_include_directories(opmcapi PUBLIC include)
target_link_libraries(opmcapi PRIVATE opmmachine opmcore)
if(MSVC)
  target_compile_definitions(opmcapi PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()

# The WPF shell picks the DLL up from shell/native/ (gitignored).
if(WIN32)
  add_custom_command(TARGET opmcapi POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:opmcapi>
            ${CMAKE_SOURCE_DIR}/shell/native/opmcapi.dll)
endif()
