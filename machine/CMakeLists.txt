add_library(opmmachine STATIC
  src/gossamer.cpp
)

target_include_directories(opmmachine PUBLIC include)
target_link_libraries(opmmachine PUBLIC opmcore)
