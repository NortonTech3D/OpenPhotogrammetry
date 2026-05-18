add_library(op_core STATIC
  src/core.cpp
)

add_library(OpenPhotogrammetry::core ALIAS op_core)

target_include_directories(op_core
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
