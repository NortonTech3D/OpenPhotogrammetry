add_library(op_io STATIC
  src/io.cpp
)

add_library(OpenPhotogrammetry::io ALIAS op_io)

target_include_directories(op_io
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(op_io
  PUBLIC
    OpenPhotogrammetry::core
)
