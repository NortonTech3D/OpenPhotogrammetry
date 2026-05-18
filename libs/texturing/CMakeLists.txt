add_library(op_texturing STATIC
  src/texturing.cpp
)

add_library(OpenPhotogrammetry::texturing ALIAS op_texturing)

target_include_directories(op_texturing
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(op_texturing
  PUBLIC
    OpenPhotogrammetry::io
    OpenPhotogrammetry::meshing
)
