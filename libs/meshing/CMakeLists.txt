add_library(op_meshing STATIC
  src/meshing.cpp
)

add_library(OpenPhotogrammetry::meshing ALIAS op_meshing)

target_include_directories(op_meshing
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(op_meshing
  PUBLIC
    OpenPhotogrammetry::mvs
)
