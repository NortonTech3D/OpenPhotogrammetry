add_library(op_mvs STATIC
  src/mvs.cpp
)

add_library(OpenPhotogrammetry::mvs ALIAS op_mvs)

target_include_directories(op_mvs
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(op_mvs
  PUBLIC
    OpenPhotogrammetry::sfm
)
