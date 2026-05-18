add_library(op_sfm STATIC
  src/sfm.cpp
)

add_library(OpenPhotogrammetry::sfm ALIAS op_sfm)

target_include_directories(op_sfm
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(op_sfm
  PUBLIC
    OpenPhotogrammetry::features
)
