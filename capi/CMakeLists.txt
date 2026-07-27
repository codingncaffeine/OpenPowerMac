add_library(opmcapi SHARED
  src/capi.cpp
)

target_include_directories(opmcapi PUBLIC include)
target_link_libraries(opmcapi PRIVATE opmmachine opmcore)
if(MSVC)
  target_compile_definitions(opmcapi PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()
